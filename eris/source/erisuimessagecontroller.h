#pragma once

#include "vstgui/lib/iviewlistener.h"
#include "vstgui/uidescription/icontroller.h"

//------------------------------------------------------------------------
namespace Steinberg {
namespace Vst {

template <typename ControllerType>
class ErisUIMessageController : public VSTGUI::IController, public VSTGUI::ViewListenerAdapter {
   public:
    enum Tags { kSendMessageTag = 1000 };

    ErisUIMessageController(ControllerType* erisController) : erisController(erisController), textEdit(nullptr) {}
    ~ErisUIMessageController() override {
        if (textEdit)
            viewWillDelete(textEdit);
        erisController->removeUIMessageController(this);
    }

    void setMessageText(String128 msgText) {
        if (!textEdit)
            return;
        String str(msgText);
        str.toMultiByte(kCP_Utf8);
        textEdit->setText(str.text8());
    }

   private:
    using CControl = VSTGUI::CControl;
    using CView = VSTGUI::CView;
    using CTextEdit = VSTGUI::CTextEdit;
    using UTF8String = VSTGUI::UTF8String;
    using UIAttributes = VSTGUI::UIAttributes;
    using IUIDescription = VSTGUI::IUIDescription;

    void valueChanged(CControl*) override {}
    void controlBeginEdit(CControl*) override {}
    void controlEndEdit(CControl* pControl) override {
        if (pControl->getTag() == kSendMessageTag) {
            if (pControl->getValueNormalized() > 0.5f) {
                erisController->sendTextMessage(textEdit->getText().data());
                pControl->setValue(0.f);
                pControl->invalid();

                if (IPtr<IMessage> message = owned(erisController->allocateMessage())) {
                    message->setMessageID("BinaryMessage");
                    uint32 size = 100;
                    char8 data[100];
                    memset(data, 0, size * sizeof(char));
                    for (uint32 i = 0; i < size; i++)
                        data[i] = i;
                    message->getAttributes()->setBinary("MyData", data, size);
                    erisController->sendMessage(message);
                }
            }
        }
    }

    CView* verifyView(CView* view, const UIAttributes&, const IUIDescription*) override {
        if (CTextEdit* te = dynamic_cast<CTextEdit*>(view)) {
            textEdit = te;

            textEdit->registerViewListener(this);

            String str(erisController->getDefaultMessageText());
            str.toMultiByte(kCP_Utf8);
            textEdit->setText(str.text8());
        }
        return view;
    }

    void viewWillDelete(CView* view) override {
        if (dynamic_cast<CTextEdit*>(view) == textEdit) {
            textEdit->unregisterViewListener(this);
            textEdit = nullptr;
        }
    }

    void viewLostFocus(CView* view) override {
        if (dynamic_cast<CTextEdit*>(view) == textEdit) {
            const UTF8String& text = textEdit->getText();
            String128 messageText;
            String str;
            str.fromUTF8(text.data());
            str.copyTo(messageText, 0, 128);
            erisController->setDefaultMessageText(messageText);
        }
    }
    ControllerType* erisController;
    CTextEdit* textEdit;
};

}  // namespace Vst
}  // namespace Steinberg
